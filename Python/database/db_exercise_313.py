"""
데이터베이스 연습 #313
SQLite CRUD 실습
"""
import sqlite3

def setup_313():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_313 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_313_0', 477.35, 'B'), ('item_313_1', 414.78, 'A'), ('item_313_2', 768.17, 'C'), ('item_313_3', 649.42, 'B'), ('item_313_4', 380.72, 'B'), ('item_313_5', 454.05, 'B'), ('item_313_6', 381.89, 'C'), ('item_313_7', 964.5, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_313 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_313()
    cursor = conn.cursor()

    print(f"DB 연습 #313")
    cursor.execute('SELECT * FROM items_313')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_313 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
