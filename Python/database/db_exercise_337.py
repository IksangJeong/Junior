"""
데이터베이스 연습 #337
SQLite CRUD 실습
"""
import sqlite3

def setup_337():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_337 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_337_0', 968.74, 'C'), ('item_337_1', 896.55, 'C'), ('item_337_2', 597.88, 'A'), ('item_337_3', 118.28, 'A'), ('item_337_4', 661.03, 'B'), ('item_337_5', 174.68, 'C'), ('item_337_6', 53.85, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_337 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_337()
    cursor = conn.cursor()

    print(f"DB 연습 #337")
    cursor.execute('SELECT * FROM items_337')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_337 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
