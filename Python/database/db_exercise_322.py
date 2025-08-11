"""
데이터베이스 연습 #322
SQLite CRUD 실습
"""
import sqlite3

def setup_322():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_322 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_322_0', 268.17, 'C'), ('item_322_1', 583.35, 'B'), ('item_322_2', 302.24, 'A'), ('item_322_3', 336.57, 'B'), ('item_322_4', 574.09, 'A'), ('item_322_5', 504.31, 'C'), ('item_322_6', 944.03, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_322 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_322()
    cursor = conn.cursor()

    print(f"DB 연습 #322")
    cursor.execute('SELECT * FROM items_322')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_322 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
